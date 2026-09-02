#!/usr/bin/env bash
# fire_bake.sh — bake the fire row ONE BUILDING PER KIT PROCESS, sequentially.
#
#     scene_gen/tools/fire_bake.sh [--dry-run] [--verify-only] [entry ...]
#
#     # the eight-building row: six GAC at F1..F6 + two MCE at F5c
#     scene_gen/tools/fire_bake.sh \
#       gac:SM_Building_02:F1 gac:SM_Building_24:F2 gac:SM_Building_01:F3 \
#       gac:SM_Building_04:F4 gac:SM_Building_06_Small:F5 gac:SM_Building_09:F6 \
#       kit:commercial_mid:F5c::S,E kit:office_wide:F5c::S,E
#
#     # just print what would run
#     scene_gen/tools/fire_bake.sh --dry-run
#
#     # re-check bakes that already exist, no Kit, no GPU
#     scene_gen/tools/fire_bake.sh --verify-only
#
# ENTRY FORMAT  `kind:name:level[:origin[:sides[:seed]]]`, empty fields absent
#   gac:SM_Building_02:F1                a merged GreatAmericanCity asset,
#                                        sliced (disaster/gac_fire.burn_gac)
#   dtc:Amar_Tower:F3                    a merged downtowncity asset, sliced
#                                        and burnt by the SAME burn_gac —
#                                        `gac_fire.PACKS` holds the pack's
#                                        directory, `.usdc` extension, unit
#                                        scale (mpu=1, not GAC's 0.01) and
#                                        construction-type source
#   kit:commercial_mid:F5c::S,E          a ModernCityEnvironment kit style
#                                        (urban_fire.burn_building), fire
#                                        pinned to the S and E elevations
#   gac:SM_Building_09:F6:3              origin storey pinned to 3
#
#     # the downtowncity row (6 blocks) beside two partially-collapsed GAC
#     scene_gen/tools/fire_bake.sh \
#       dtc:Building_11:F1 dtc:Building_12:F2 dtc:Carved_18:F3 \
#       dtc:Carved_14:F4 dtc:Carved_13:F5 dtc:Amar_Tower:F3 \
#       gac:SM_Building_02:F5c gac:SM_Building_24:F5c
#
# WHY ONE PROCESS PER BUILDING
# ----------------------------
# The combined bench (`gac_fire_bench_launch_script.py`) built the whole row
# in ONE process: 25 GB RSS, a single settle of ~2,350 loose bodies that ended
# with 688 STILL MOVING at bake time, and a slow scene open. A per-building
# bake bounds memory to one building, gives the settle the whole step budget
# so it can actually converge, and leaves the assembly a fast static load
# (user, 2026-08-30: "you can do 1 building at a time, bake it then launch
# them together as static").
#
# ASSUMPTIONS — READ BEFORE RUNNING
# ---------------------------------
# * The `isaac-sim` container is UP. This driver does NOT touch its tmux
#   pane: each bake is a separate headless `docker exec` process (the pattern
#   `scene_gen/tools/eq_bench.sh` established), so it is safe to run while
#   somebody has the GUI pane open and idle. It is NOT safe to run while the
#   pane is itself building a scene — one 16 GB card does not hold two of
#   these — and `gac_storey_slice` takes a MACHINE-WIDE LOCK, so a live slice
#   in the pane will simply block this one.
# * SEQUENTIAL ON PURPOSE, one bake at a time, for that same lock and for
#   memory. Do not add a `&`.
# * The bakes land in the CONTAINER at `$FB_OUT`
#   (default /isaac-sim/.cache/fire_bakes), not in the repo: "I don't need
#   these bakes to be saved on host since we will need to work on them, they
#   can be container only" (user, 2026-08-30). The soot PNGs go with them,
#   under `$FB_OUT/textures`.
# * Each bake's stdout is `$HOME/docker/isaac-sim/logs/<stem>.log` on the
#   host (= /isaac-sim/.nvidia-omniverse/logs/<stem>.log in the container).
#   Read THAT — `docker logs isaac-sim` is empty for this container.
#
# Env (all optional):
#   FB_OUT           container output dir      (default /isaac-sim/.cache/fire_bakes)
#   FB_SEED          base seed; building i gets FB_SEED + 31*i for the burn
#                    and FB_SEED + 7*i for `build_building`, which is exactly
#                    what `gac_fire_bench_launch_script.py` gives column i
#                                              (default 7)
#   SETTLE_STEPS     step target per building, F5/F5c/F6 tier (default 2400)
#   SETTLE_QUIET     quiet-phase steps, F5/F5c/F6 tier        (default 400)
#   SETTLE_STEPS_LOW step target, F0-F3 tier   (default 600) -- see
#   SETTLE_QUIET_LOW quiet-phase steps, F0-F3 tier (default 150) -- PER-LEVEL
#   SETTLE_STEPS_MID step target, F4 tier      (default 1600) -- SETTLE
#   SETTLE_QUIET_MID quiet-phase steps, F4 tier (default 300) -- BUDGETS below
#   FB_LEVEL_SETTLE  1 (default) applies the tiering above per entry's own
#                    LEVEL; 0 is the flat pre-2026-08-31 behaviour (every
#                    entry gets SETTLE_STEPS/SETTLE_QUIET regardless of level)
#   SETTLE_DECOMP_M  convex-decomposition threshold, m (default 0.8)
#   SETTLE_FABRIC    1 keeps PhysX transforms in Fabric during the settle
#                    (no per-step USD write-back; read back once at the end)
#                                              (default 0)
#   FB_BAKED_KITS    use pre-baked GAC kits    (default 1)
#   SOOT_TEX_COMPRESS  1 (default, set inside the container -- NOT here) bakes
#                    soot textures as BC1 DDS instead of PNG (~8x less VRAM,
#                    same look -- `disaster/tex_compress.py`). Left UNSET by
#                    this driver unless the caller exports it, so the
#                    container-side default applies; export SOOT_TEX_COMPRESS=0
#                    before running this script for an A/B baseline bake.
#   TIMEOUT_S        per-building ceiling      (default 5400)
#   CONTAINER        container name            (default isaac-sim)
#   REPO             repo path in container    (default /isaac-sim/AirStack)
#
# PER-LEVEL SETTLE BUDGETS (2026-08-31, time-crunch directive: "make sure
# the bakes are all lazy"). `urban_fire.LADDER` only puts loose bodies in the
# pile from `fire_collapse`/`partial_collapse`/`floor_burnthrough`/
# `roof_burnthrough` — `street_debris` is hand-placed straight into
# `authored`, never `loose`. F0/F1 have NONE of those four in either
# construction type; F2/F3 have at most one (roof_burnthrough, urm only, up
# to frac 0.20-0.34); F4 is a real mix (0-144 loose bodies measured across
# the last sweep's F4 records); F5/F5c/F6 ALWAYS run a collapse recipe and
# routinely settle 400-600+ bodies, so that tier keeps `SETTLE_STEPS`/
# `SETTLE_QUIET` exactly as before — "a level with real debris keeps its
# full settle". Below that, `settle.run`'s own chunked `_settle_phase`
# ALREADY exits the instant a chunk finds nothing moving (`converge=True`
# only ever makes `steps`/`quiet_steps` a FLOOR it may run past, up to
# `max_steps=3*steps` — see settle.py's docstring), so a lower tier is not
# "cut the settle short and hope": it shrinks the CHUNK SIZE
# (`max(30, steps//12)`) so that poll fires sooner, and it lowers the
# worst-case ceiling for a level that is supposed to have almost nothing to
# settle. Margins were sized off the last city sweep's sidecars: LOW tier's
# worst observed `steps_used` was 1200 (of a 7200 cap) on an F3 kit record,
# MID tier's was 1600 (of 7200) on an F4 kit record — the new ceilings (3x
# the tier's own target) sit 1.5-3x above both.

set -u

CONTAINER=${CONTAINER:-isaac-sim}
REPO=${REPO:-/isaac-sim/AirStack}
FB_OUT=${FB_OUT:-/isaac-sim/.cache/fire_bakes}
FB_SEED=${FB_SEED:-7}
SETTLE_STEPS=${SETTLE_STEPS:-2400}
SETTLE_QUIET=${SETTLE_QUIET:-400}
SETTLE_STEPS_LOW=${SETTLE_STEPS_LOW:-600}
SETTLE_QUIET_LOW=${SETTLE_QUIET_LOW:-150}
SETTLE_STEPS_MID=${SETTLE_STEPS_MID:-1600}
SETTLE_QUIET_MID=${SETTLE_QUIET_MID:-300}
FB_LEVEL_SETTLE=${FB_LEVEL_SETTLE:-1}
SETTLE_DECOMP_M=${SETTLE_DECOMP_M:-0.8}
SETTLE_FABRIC=${SETTLE_FABRIC:-0}      # 1 = PhysX->Fabric, no per-step USD write-back (settle.py)
SOOT_TEX_COMPRESS=${SOOT_TEX_COMPRESS:-}   # left EMPTY unless the caller set it --
                                            # see the SOOT_TEX_COMPRESS note above;
                                            # an explicit "" would force the
                                            # container-side default OFF
SETTLE_REST_V2=${SETTLE_REST_V2:-1}    # settle.py's corrected rest/grade measurements; forced OFF
#                                        for `kit:` records, whose look is frozen against the old ones
FB_BAKED_KITS=${FB_BAKED_KITS:-1}
TIMEOUT_S=${TIMEOUT_S:-5400}
LAUNCHER="$REPO/simulation/isaac-sim/launch_scripts/fire_bake_launch_script.py"
HOSTLOGDIR="$HOME/docker/isaac-sim/logs"
CLOGDIR="/isaac-sim/.nvidia-omniverse/logs"

# settle_for_level LEVEL -- sets REC_STEPS/REC_QUIET/REC_TIER for one entry.
# F0-F3 = "low", F4 = "mid", F5/F5c/F6 (and anything unrecognised, so an
# unknown level fails SAFE toward the full budget) = "high" = unchanged
# SETTLE_STEPS/SETTLE_QUIET.
settle_for_level() {
  local lvl="$1"
  if [ "$FB_LEVEL_SETTLE" = "0" ]; then
    REC_STEPS=$SETTLE_STEPS; REC_QUIET=$SETTLE_QUIET; REC_TIER="flat"
    return
  fi
  case "$lvl" in
    F0|F1|F2|F3)
      REC_STEPS=$SETTLE_STEPS_LOW; REC_QUIET=$SETTLE_QUIET_LOW; REC_TIER="low" ;;
    F4)
      REC_STEPS=$SETTLE_STEPS_MID; REC_QUIET=$SETTLE_QUIET_MID; REC_TIER="mid" ;;
    F5|F5c|F6)
      REC_STEPS=$SETTLE_STEPS; REC_QUIET=$SETTLE_QUIET; REC_TIER="high" ;;
    *)
      REC_STEPS=$SETTLE_STEPS; REC_QUIET=$SETTLE_QUIET; REC_TIER="high(unknown level $lvl)" ;;
  esac
}

DRY=0
VERIFY_ONLY=0
ENTRIES=()
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1 ;;
    --verify-only) VERIFY_ONLY=1 ;;
    -h|--help) sed -n '2,113p' "$0"; exit 0 ;;
    -*) echo "fire_bake.sh: unknown flag $1" >&2; exit 2 ;;
    *) ENTRIES+=("$1") ;;
  esac
  shift
done

# THE DEFAULT ROW is the one the combined bench built: six GAC buildings at
# F1..F6 plus the two ModernCityEnvironment kit buildings at F5c (partial
# collapse). Same assets, same order, same per-column seeds — so a bake of
# column i is that bench's building i, not a different draw of it.
if [ ${#ENTRIES[@]} -eq 0 ]; then
  ENTRIES=(gac:SM_Building_02:F1
           gac:SM_Building_24:F2
           gac:SM_Building_01:F3
           gac:SM_Building_04:F4
           gac:SM_Building_06_Small:F5
           gac:SM_Building_09:F6
           kit:commercial_mid:F5c::S,E
           kit:office_wide:F5c::S,E)
fi

if [ "$DRY" = 0 ] && ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "fire_bake.sh: container $CONTAINER is not running" >&2
  exit 1
fi
mkdir -p "$HOSTLOGDIR" 2>/dev/null || true

# ---------------------------------------------------------------------------
# --verify-only: reopen every bake COLD in the bare-USD harness and check its
# materials resolve, no prim still points into the dropped source subtree, no
# physics API survived and no Flow prim shipped. No Kit, no GPU, safe beside
# a live session (`scene_gen/tools/usd_python.sh`).
# ---------------------------------------------------------------------------
if [ "$VERIFY_ONLY" = 1 ]; then
  CMD="$REPO/scene_gen/tools/usd_python.sh $REPO/scene_gen/tests/test_fire_bake.py --verify $FB_OUT"
  echo "+ docker exec $CONTAINER bash -c \"$CMD\""
  [ "$DRY" = 1 ] && exit 0
  docker exec "$CONTAINER" bash -c "$CMD"
  exit $?
fi

echo "fire_bake.sh: ${#ENTRIES[@]} building(s) -> $CONTAINER:$FB_OUT"
echo "              seed base $FB_SEED, settle (F5/F5c/F6) ${SETTLE_STEPS}+${SETTLE_QUIET}$([ "$FB_LEVEL_SETTLE" != 0 ] && echo ", (F0-F3) ${SETTLE_STEPS_LOW}+${SETTLE_QUIET_LOW}, (F4) ${SETTLE_STEPS_MID}+${SETTLE_QUIET_MID}"), decomp ${SETTLE_DECOMP_M} m"
echo

t_all=$(date +%s)
i=0
ok=0
fail=0
declare -a SUMMARY=()
declare -a USDS=()
for ent in "${ENTRIES[@]}"; do
  IFS=':' read -r KIND NAME LEVEL ORIGIN SIDES SEED <<EOF
$ent
EOF
  LEVEL=${LEVEL:-F3}
  ORIGIN=${ORIGIN:-}
  SIDES=${SIDES:-}
  if [ -z "${SEED:-}" ]; then SEED=$(( FB_SEED + 31 * i )); fi
  BUILD_SEED=$(( FB_SEED + 7 * i ))
  STEM="${KIND}_${NAME}_${LEVEL}_s${SEED}"
  CLOG="$CLOGDIR/$STEM.log"
  HLOG="$HOSTLOGDIR/$STEM.log"

  # PYTHONHASHSEED IS NOT COSMETIC HERE. `urban_fire.r_render_peel` does
  # `for side in sides_hit:` over a SET of side letters, and Python
  # randomises `str` hashes per PROCESS, so that loop runs E/W/S in a
  # different order every bake — which hands the shared `rng` a different
  # draw order from that point on and changes every debris placement after
  # it. Measured 2026-08-31 on `kit_brownstone_row_F5c_o4_SEW_s198`: two
  # bakes of the SAME record with the SAME code moved 326 meshes, mirrored
  # the `peelrow` windrow 40 m end-for-end, and changed top_z 21.69 -> 20.76
  # m. Pinning the seed makes a bake reproducible; it does NOT make it match
  # a bake taken before this line existed.
  ENVS="ISAAC_SIM_HEADLESS=true PYTHONUNBUFFERED=1 PYTHONHASHSEED=0"
  ENVS="$ENVS FB_KIND=$KIND FB_NAME=$NAME FB_LEVEL=$LEVEL"
  ENVS="$ENVS FB_SEED=$SEED FB_BUILD_SEED=$BUILD_SEED FB_INDEX=$i"
  ENVS="$ENVS FB_ORIGIN=$ORIGIN FB_SIDES=$SIDES"
  ENVS="$ENVS FB_OUT=$FB_OUT FB_BAKED_KITS=$FB_BAKED_KITS FB_VERIFY=1"
  # only forwarded when the caller actually set it — see the declaration
  # above for why an unconditional "SOOT_TEX_COMPRESS=$SOOT_TEX_COMPRESS"
  # would silently force the container-side default OFF
  [ -n "$SOOT_TEX_COMPRESS" ] && ENVS="$ENVS SOOT_TEX_COMPRESS=$SOOT_TEX_COMPRESS"
  # PER-LEVEL SETTLE BUDGET — see the header comment.
  settle_for_level "$LEVEL"
  ENVS="$ENVS SETTLE_STEPS=$REC_STEPS SETTLE_QUIET=$REC_QUIET"
  ENVS="$ENVS SETTLE_DECOMP_M=$SETTLE_DECOMP_M SETTLE_FABRIC=$SETTLE_FABRIC"
  REST_V2=$SETTLE_REST_V2
  [ "$KIND" = "kit" ] && REST_V2=0   # the MCE kit look is FROZEN -- see settle.py
  ENVS="$ENVS SETTLE_REST_V2=$REST_V2"
  RUN="mkdir -p '$CLOGDIR' '$FB_OUT'; : > '$CLOG'; cd /isaac-sim && env $ENVS PYTHONPATH=\"\$ISAAC_SIM_PYTHONPATH\" timeout ${TIMEOUT_S}s /isaac-sim/python.sh $LAUNCHER --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts --no-window > '$CLOG' 2>&1; echo \"EXIT \$?\" >> '$CLOG'"

  echo "=== [$i] $ent -> $STEM  settle=$REC_TIER(${REC_STEPS}+${REC_QUIET})"
  echo "+ docker exec $CONTAINER bash -c \"$RUN\""
  if [ "$DRY" = 1 ]; then USDS+=("$FB_OUT/$STEM.usd"); i=$((i+1)); continue; fi

  t0=$(date +%s)
  docker exec "$CONTAINER" bash -c "$RUN"
  dt=$(( $(date +%s) - t0 ))

  if docker exec "$CONTAINER" grep -q "FIRE BAKE DONE" "$CLOG" 2>/dev/null; then
    SZ=$(docker exec "$CONTAINER" bash -c "stat -c %s '$FB_OUT/$STEM.usd' 2>/dev/null || echo 0")
    MB=$(awk -v b="$SZ" 'BEGIN{printf "%.1f", b/1e6}')
    VOK=$(docker exec "$CONTAINER" grep -c "BAKE VERIFY OK" "$CLOG" 2>/dev/null || echo 0)
    STILL=$(docker exec "$CONTAINER" grep -o "still moving [0-9]*" "$CLOG" 2>/dev/null | tail -1)
    echo "    DONE in ${dt}s, ${MB} MB, verify=$([ "$VOK" -gt 0 ] && echo OK || echo PROBLEM) ${STILL:-}"
    SUMMARY+=("$(printf '%-2s %-34s %-5s %7ss %8s MB  %s' "$i" "$STEM" "$LEVEL" "$dt" "$MB" "$([ "$VOK" -gt 0 ] && echo verify-OK || echo VERIFY-PROBLEM)")")
    # ONLY WHAT ACTUALLY BAKED goes in the assembly list: a failed building
    # would otherwise become a dangling reference and an empty column.
    USDS+=("$FB_OUT/$STEM.usd")
    ok=$((ok+1))
  else
    echo "    FAILED after ${dt}s — see $HLOG"
    docker exec "$CONTAINER" grep -E "Traceback|Error|FIRE BAKE FAILED|EXIT [1-9]" "$CLOG" 2>/dev/null | tail -12
    SUMMARY+=("$(printf '%-2s %-34s %-5s %7ss %8s     FAILED' "$i" "$STEM" "$LEVEL" "$dt" "-")")
    fail=$((fail+1))
  fi
  i=$((i+1))
done

[ "$DRY" = 1 ] && { echo; echo "(dry run — nothing was executed)"; }

echo
echo "========================================================================"
echo "FIRE BAKE ROW   $ok ok, $fail failed, $(( $(date +%s) - t_all )) s total"
if [ ${#SUMMARY[@]} -gt 0 ]; then printf '%s\n' "${SUMMARY[@]}"; fi
echo "  bakes:    $CONTAINER:$FB_OUT"
echo "  textures: $CONTAINER:$FB_OUT/textures"
echo "  logs:     $HOSTLOGDIR/<stem>.log"
echo "------------------------------------------------------------------------"
# NAME THE FILES, NOT THE DIRECTORY. `$FB_OUT` accumulates: a re-bake at a
# different FB_SEED leaves the old stems in place and the assembly would put
# sixteen columns in the row. An explicit FA_BAKES list is what this run
# actually produced.
FA_LIST=$(IFS=,; echo "${USDS[*]-}")
[ -z "$FA_LIST" ] && FA_LIST="$FB_OUT"
echo 'Assemble them (GUI, keeps the app open). "docker exec -e" rather than a'
echo 'nested "env ..." so nothing has to survive two layers of shell quoting:'
echo
echo "  docker exec \\"
echo "    -e ISAAC_SIM_HEADLESS=false -e PYTHONUNBUFFERED=1 \\"
echo "    -e FA_BAKES=$FA_LIST -e FA_FLOW=1 -e KEEP_OPEN=1 \\"
echo "    -e SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/fire_row \\"
echo "    $CONTAINER bash -lc 'cd /isaac-sim && PYTHONPATH=\"\$ISAAC_SIM_PYTHONPATH\" \\"
echo "      /isaac-sim/python.sh \\"
echo "      $REPO/simulation/isaac-sim/launch_scripts/fire_assembly_launch_script.py \\"
echo "      --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts'"
echo "========================================================================"
[ "$fail" = 0 ]
