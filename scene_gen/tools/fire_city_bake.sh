#!/usr/bin/env bash
# fire_city_bake.sh — lazy, cache-checked bake of a CITY fire manifest, one
# building per Kit process, sequentially, over `fire_bake.sh`'s own
# per-building semantics (`urban_fire_city_plan.md` sec 3, work item #6 of
# its `## 6. Work breakdown`). Does NOT fork `fire_bake.sh` — this is a
# thin driver that reuses its `docker exec` line shape and env table, adds
# the cache check `fire_bake.sh` never needed (a row is short enough to
# always rebake), and points each bake at a CITY-scoped output directory
# plus a per-record `FB_CITY_JSON` sidecar payload.
#
#     scene_gen/tools/fire_city_bake.sh MANIFEST [--dry-run] [--verify-only] [--force]
#
#     # just print the plan (classification + the docker exec lines that
#     # would run), touch nothing
#     scene_gen/tools/fire_city_bake.sh _plans/fire_city_1013.json --dry-run
#
#     # bake whatever is missing, skip what is already there
#     scene_gen/tools/fire_city_bake.sh _plans/fire_city_1013.json
#
#     # rebake everything regardless of cache state
#     scene_gen/tools/fire_city_bake.sh _plans/fire_city_1013.json --force
#
#     # re-check every bake already in the city dir, no Kit, no GPU
#     scene_gen/tools/fire_city_bake.sh _plans/fire_city_1013.json --verify-only
#
# MANIFEST — see `scene_gen/tools/fire_city_manifest.py`'s own docstring
# for the exact JSON shape (a bare array of `urban_fire_city.damaged_manifest`
# records, or an object with that array under manifest/records/buildings/
# entries and an optional top-level `"seed"`). This driver never parses the
# manifest itself — `fire_city_manifest.py` (host python3, no pxr) is the
# ONE place that turns a record into an entry string + cache stem, so a
# bake this driver skips and a bake `fire_city_manifest.py --write-city-json`
# describes are always talking about the exact same stem.
#
# CACHE KEY = the stem `fire_bake.out_stem` computes for the record's entry
# (kind_name_level[_oORIGIN][_SIDES]_sSEED). A record is HAVE (skipped,
# unless --force) when both `<stem>.usd` and `<stem>.json` already exist
# under the CITY output dir — existence only, never re-verified here (that
# is what `--verify-only` and `fire_city_manifest.py --verify` are for);
# "otherwise trust existence" is this whole pipeline's own stated brief.
#
# WHERE BAKES LAND. `FB_OUT` (default /isaac-sim/.cache/fire_bakes, the
# CONTAINER path, exactly `fire_bake.sh`'s own default) gets a CITY
# subdirectory appended: every bake this run produces or finds goes to
# `$FB_OUT/city_<seed>/`, never the bare `$FB_OUT` a plain row bake uses —
# a city and a row sharing one flat directory would silently reuse (or
# collide with) each other's stems, and a second city at a different seed
# would otherwise pile its columns into the first one's assembly list
# forever (the exact accumulation trap `fire_bake.sh`'s own summary already
# warns about for FB_OUT in general). `<seed>` is `fire_city_manifest.py
# --print-seed`'s answer: the manifest's own top-level `"seed"`, or the
# trailing digits of its filename.
#
# THE HOST/CONTAINER SPLIT. The classification (HAVE/NEED) is a plain
# filesystem existence check, and it has to run against a path THIS PROCESS
# (a host bash script) can actually see — but the bake itself always runs
# in the CONTAINER, addressing `$FB_OUT` as a container path, exactly like
# `fire_bake.sh`. So this driver keeps TWO forms of the city output dir,
# the same way `fire_bake.sh` already keeps HOSTLOGDIR (`$HOME/docker/
# isaac-sim/logs`) beside CLOGDIR (`/isaac-sim/.nvidia-omniverse/logs`) for
# its own per-bake log:
#   * CITY_OUT       — the CONTAINER path, `$FB_OUT/city_<seed>` — what
#                      gets exported as `FB_OUT=` for the actual bake.
#   * CITY_OUT_HOST  — CITY_OUT with the known container->host cache
#                      prefix substituted (`/isaac-sim/.cache` ->
#                      `$HOME/docker/isaac-sim/cache/main`, straight out of
#                      `simulation/isaac-sim/docker/docker-compose.yaml`'s
#                      bind mount) — what `fire_city_manifest.py --out-dir`
#                      is actually pointed at. When `FB_OUT` does not start
#                      with the known container prefix (a custom path, or a
#                      test's own plain tmp dir), NO translation happens
#                      and CITY_OUT_HOST == CITY_OUT — the same tmp
#                      directory is then both the "container" and "host"
#                      view, which is exactly what lets the smoke test
#                      (`scene_gen/tests/test_fire_city_bake.sh`) exercise
#                      real classification with no docker/container in the
#                      loop at all.
# The per-record `FB_CITY_JSON` sidecar payload gets the identical
# treatment against `$REPO` (default /isaac-sim/AirStack) <-> this repo's
# own checkout root, since it is written under the repo (which the
# container bind-mounts 1:1 at `$REPO`) rather than under the cache.
#
# Env (all optional, same names as `fire_bake.sh` where they overlap):
#   FB_OUT           container output BASE dir  (default /isaac-sim/.cache/fire_bakes)
#                    -- the city subdir `city_<seed>` is appended by this script
#   FB_SEED          base seed for BUILD_SEED (`FB_SEED + 7*i`), NOT the
#                    per-building burn seed (that is the manifest record's
#                    own `seed`, already baked into its entry string)
#                                              (default 7)
#   SETTLE_STEPS, SETTLE_QUIET, SETTLE_DECOMP_M, SETTLE_FABRIC, FB_BAKED_KITS,
#   SOOT_TEX_COMPRESS, TIMEOUT_S, CONTAINER, REPO -- identical to fire_bake.sh
#   (SOOT_TEX_COMPRESS: left unset here too unless the caller exports it, so
#   the container-side default -- BC1 DDS soot textures, ON -- applies; the
#   VRAM pass this driver now folds in, `disaster/tex_compress.py`)
#   SETTLE_REST_V2   1 (default) turns on `disaster.settle`'s corrected rest
#                    and grade measurements FOR NON-`kit` RECORDS ONLY. The
#                    MCE kit look is frozen against the old (box-based
#                    below-grade, per-chunk still-moving) behaviour, so a
#                    `kit:` record is always baked with it OFF and reproduces
#                    byte for byte. Set 0 to bake everything the old way.
#   FB_REST_STRICT   1 makes a record that did not come to rest a FAILURE
#                    (non-zero exit) instead of a loud line.   (default 0)
#
# PER-LEVEL SETTLE BUDGETS (2026-08-31, time-crunch directive: "make sure
# the bakes are all lazy"). `urban_fire.LADDER` only puts loose bodies in the
# pile from `fire_collapse`/`partial_collapse`/`floor_burnthrough`/
# `roof_burnthrough` -- `street_debris` is hand-placed straight into
# `authored`, never `loose`. F0/F1 have NONE of those four in either
# construction type; F2/F3 have at most one (roof_burnthrough, urm only, and
# only up to frac 0.20-0.34); F4 is a real mix (0-144 loose bodies measured
# across the last sweep's F4 records, `rc`/`rc_glass` often add nothing, urm
# and a lucky draw can add a lot); F5/F5c/F6 ALWAYS run a collapse recipe and
# routinely settle 400-600+ bodies. `SETTLE_STEPS`/`SETTLE_QUIET` above are
# reused AS-IS for the F5/F5c/F6 tier (unchanged: 2400/400) -- "a level with
# real debris keeps its full settle". Below that, `settle.run`'s own chunked
# `_settle_phase` ALREADY exits the instant a chunk finds nothing moving
# (`converge=True` only ever makes `steps`/`quiet_steps` a FLOOR it may run
# past, up to `max_steps=3*steps` -- see settle.py's docstring), so a lower
# tier is not "cut the settle short and hope" — it shrinks the CHUNK SIZE
# (`max(30, steps//12)`) so the poll that finds "nothing moving" fires
# sooner, and it lowers the worst-case ceiling for a level that is supposed
# to have almost nothing to settle. Margins were sized off the last sweep's
# sidecars (`counts.json`/`settle` fields): LOW tier's worst observed
# `steps_used` was 1200 (of a 7200 cap) on an F3 kit record, MID tier's was
# 1600 (of 7200) on an F4 kit record -- the new ceilings (`3x` the tier's own
# target) sit 1.5-3x above both, so a heavier-than-typical draw still has
# room to converge rather than getting capped mid-pile.
#   SETTLE_STEPS_LOW   F0/F1/F2/F3 throw-phase step TARGET   (default 600)
#   SETTLE_QUIET_LOW   F0/F1/F2/F3 quiet-phase step TARGET   (default 150)
#   SETTLE_STEPS_MID   F4 throw-phase step TARGET            (default 1600)
#   SETTLE_QUIET_MID   F4 quiet-phase step TARGET            (default 300)
#   FB_LEVEL_SETTLE    1 (default) applies the tiering above per record's own
#                      `FB_LEVEL`. 0 goes back to the flat pre-2026-08-31
#                      behaviour: every record gets `SETTLE_STEPS`/
#                      `SETTLE_QUIET` regardless of level (useful for an
#                      apples-to-apples timing comparison, or if a level's
#                      tier guess above ever needs to be pinned back to full).
#
# ASSUMPTIONS — same as fire_bake.sh: the isaac-sim container is UP for a
# real bake (not for --dry-run / --verify-only), one bake at a time on
# purpose (the machine-wide slicer lock, one GPU), this script never
# touches tmux and never starts anything but a headless `docker exec`.

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_HOST="$(cd "$HERE/../.." && pwd)"

CONTAINER=${CONTAINER:-isaac-sim}
REPO=${REPO:-/isaac-sim/AirStack}
FB_OUT=${FB_OUT:-/isaac-sim/.cache/fire_bakes}
FB_SEED=${FB_SEED:-7}
SETTLE_STEPS=${SETTLE_STEPS:-2400}
SETTLE_QUIET=${SETTLE_QUIET:-400}
SETTLE_DECOMP_M=${SETTLE_DECOMP_M:-0.8}
SETTLE_FABRIC=${SETTLE_FABRIC:-0}
SOOT_TEX_COMPRESS=${SOOT_TEX_COMPRESS:-}   # left EMPTY unless the caller set it --
                                            # an explicit "" would force the
                                            # container-side default OFF
SETTLE_REST_V2=${SETTLE_REST_V2:-1}
FB_REST_STRICT=${FB_REST_STRICT:-0}
# Per-level settle budgets -- see the header comment. F5/F5c/F6 keep the
# SETTLE_STEPS/SETTLE_QUIET values above unchanged (the tier with real
# debris); F0-F3 and F4 get their own, smaller targets below.
SETTLE_STEPS_LOW=${SETTLE_STEPS_LOW:-600}
SETTLE_QUIET_LOW=${SETTLE_QUIET_LOW:-150}
SETTLE_STEPS_MID=${SETTLE_STEPS_MID:-1600}
SETTLE_QUIET_MID=${SETTLE_QUIET_MID:-300}
FB_LEVEL_SETTLE=${FB_LEVEL_SETTLE:-1}
FB_BAKED_KITS=${FB_BAKED_KITS:-1}
TIMEOUT_S=${TIMEOUT_S:-5400}
LAUNCHER="$REPO/simulation/isaac-sim/launch_scripts/fire_bake_launch_script.py"
HOSTLOGDIR="$HOME/docker/isaac-sim/logs"
CLOGDIR="/isaac-sim/.nvidia-omniverse/logs"
MANIFEST_PY="$HERE/fire_city_manifest.py"

# known bind-mount pairs (host prefix <-> container prefix) -- see the
# header comment. Order matters only in that longer/more specific prefixes
# should be checked first; there is exactly one of each here.
CACHE_CONTAINER_PREFIX="/isaac-sim/.cache"
CACHE_HOST_PREFIX="$HOME/docker/isaac-sim/cache/main"

to_host_path() {
  # `$1` a path that may start with the known CONTAINER cache prefix ->
  # its host-mounted equivalent; anything else is returned unchanged (the
  # "no translation" fallback the smoke test relies on).
  local p="$1"
  case "$p" in
    "$CACHE_CONTAINER_PREFIX"*)
      printf '%s%s\n' "$CACHE_HOST_PREFIX" "${p#$CACHE_CONTAINER_PREFIX}" ;;
    *) printf '%s\n' "$p" ;;
  esac
}

to_container_repo_path() {
  # host repo-relative path -> its container form under `$REPO`.
  local p="$1"
  printf '%s%s\n' "$REPO" "${p#$REPO_HOST}"
}

# settle_for_level LEVEL -- sets REC_STEPS/REC_QUIET/REC_TIER for one record.
# See the header comment (PER-LEVEL SETTLE BUDGETS) for why F0-F3 are "low",
# F4 is "mid", and F5/F5c/F6 (and anything this ladder does not know, so an
# unrecognised level fails SAFE toward the full budget rather than toward a
# possibly-too-small guess) keep the SETTLE_STEPS/SETTLE_QUIET value as-is.
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
FORCE=0
MANIFEST=""
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1 ;;
    --verify-only) VERIFY_ONLY=1 ;;
    --force) FORCE=1 ;;
    -h|--help) sed -n '2,139p' "$0"; exit 0 ;;
    -*) echo "fire_city_bake.sh: unknown flag $1" >&2; exit 2 ;;
    *)
      if [ -n "$MANIFEST" ]; then
        echo "fire_city_bake.sh: unexpected extra argument $1" >&2; exit 2
      fi
      MANIFEST="$1" ;;
  esac
  shift
done

if [ -z "$MANIFEST" ]; then
  echo "usage: fire_city_bake.sh MANIFEST [--dry-run] [--verify-only] [--force]" >&2
  exit 2
fi
if [ ! -f "$MANIFEST" ]; then
  echo "fire_city_bake.sh: manifest not found: $MANIFEST" >&2
  exit 2
fi
if ! command -v python3 >/dev/null 2>&1; then
  echo "fire_city_bake.sh: python3 is required (for fire_city_manifest.py)" >&2
  exit 2
fi

CITY_SEED=$(python3 "$MANIFEST_PY" "$MANIFEST" --print-seed) || exit 2
CITY_OUT="$FB_OUT/city_$CITY_SEED"
CITY_OUT_HOST=$(to_host_path "$CITY_OUT")
CITY_JSON_DIR_HOST="$REPO_HOST/scene_gen/_plans/_fire_city_json/city_$CITY_SEED"
CITY_JSON_DIR="$(to_container_repo_path "$CITY_JSON_DIR_HOST")"

echo "fire_city_bake.sh: manifest $MANIFEST, city seed $CITY_SEED"
echo "                   bakes -> $CONTAINER:$CITY_OUT  (host view: $CITY_OUT_HOST)"
echo

# ---------------------------------------------------------------------------
# --verify-only: exactly `fire_bake.sh --verify-only`'s own call, pointed at
# the city directory instead of the flat row directory. No Kit, no GPU.
# ---------------------------------------------------------------------------
if [ "$VERIFY_ONLY" = 1 ]; then
  CMD="$REPO/scene_gen/tools/usd_python.sh $REPO/scene_gen/tests/test_fire_bake.py --verify $CITY_OUT"
  echo "+ docker exec $CONTAINER bash -c \"$CMD\""
  [ "$DRY" = 1 ] && exit 0
  docker exec "$CONTAINER" bash -c "$CMD"
  exit $?
fi

if [ "$DRY" = 0 ] && ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "fire_city_bake.sh: container $CONTAINER is not running" >&2
  exit 1
fi

# ---------------------------------------------------------------------------
# Classification -- ONE call to fire_city_manifest.py, host-side, no pxr,
# no docker. `--out-dir` gets the HOST view of the city directory; the
# ENTRY string in its output already carries everything a bake needs.
# This runs even under --dry-run (it is read-only against $CITY_OUT_HOST,
# and dry-run's whole point is to show real HAVE/NEED classification).
# ---------------------------------------------------------------------------
CLASS_OUT=$(python3 "$MANIFEST_PY" "$MANIFEST" --out-dir "$CITY_OUT_HOST") || {
  echo "fire_city_bake.sh: fire_city_manifest.py classification failed" >&2
  exit 2
}

# The per-record city sidecar payload (`cell`/`x`/`y`/`yaw_deg`/`z`/
# `typology`/`orig_usd`) -- written once up front for every record
# regardless of HAVE/NEED, so a later `--force` rebake of a HAVE entry
# still finds its FB_CITY_JSON file waiting. Skipped under --dry-run --
# "prints the plan and skips" means touching nothing, including this repo
# directory; the printed FB_CITY_JSON= path below is still the real path
# a non-dry run would write.
if [ "$DRY" = 0 ]; then
  mkdir -p "$HOSTLOGDIR" "$CITY_JSON_DIR_HOST" 2>/dev/null || true
  python3 "$MANIFEST_PY" "$MANIFEST" --write-city-json "$CITY_JSON_DIR_HOST" \
    >/dev/null || { echo "fire_city_bake.sh: --write-city-json failed" >&2; exit 2; }
fi

echo "$CLASS_OUT" | grep -v '^SUMMARY' | sed 's/^/  classify: /'
echo "  classify: $(echo "$CLASS_OUT" | grep '^SUMMARY')"
echo

t_all=$(date +%s)
ok=0
skipped=0
fail=0
notrest=0
i=0
declare -a SUMMARY=()
declare -a USDS=()

while IFS=$'\t' read -r ENTRY STEM STATUS REST; do
  [ -z "$ENTRY" ] && continue
  if [ "$ENTRY" = "SUMMARY" ] || [[ "$ENTRY" == SUMMARY* ]]; then continue; fi
  if [ "$STATUS" = "ERROR" ]; then
    echo "=== [$i] SKIP (manifest record error): ${REST:-$STEM}"
    fail=$((fail+1)); i=$((i+1)); continue
  fi

  if [ "$STATUS" = "HAVE" ] && [ "$FORCE" = 0 ]; then
    echo "=== [$i] $ENTRY -> $STEM  (HAVE, skipping)"
    USDS+=("$CITY_OUT/$STEM.usd")
    SUMMARY+=("$(printf '%-2s %-42s %-6s %s' "$i" "$STEM" "-" "skip (cached)")")
    skipped=$((skipped+1)); i=$((i+1)); continue
  fi

  IFS=':' read -r KIND NAME LEVEL ORIGIN SIDES SEED <<EOF
$ENTRY
EOF
  BUILD_SEED=$(( FB_SEED + 7 * i ))
  CLOG="$CLOGDIR/city_${CITY_SEED}_$STEM.log"
  HLOG="$HOSTLOGDIR/city_${CITY_SEED}_$STEM.log"
  CITY_JSON_C="$CITY_JSON_DIR/$STEM.city.json"

  # PYTHONHASHSEED IS NOT COSMETIC HERE — WITHOUT IT A BAKE IS NOT
  # REPRODUCIBLE. `urban_fire.r_render_peel` ends with `for side in
  # sides_hit:` over a SET of side letters, and CPython randomises `str`
  # hashes per PROCESS, so that loop visits E/W/S in a different order every
  # run. Each visit draws from the SHARED `rng`, so from that point on every
  # subsequent debris placement in the build gets a different draw.
  # MEASURED 2026-08-31, two bakes of `kit:brownstone_row:F5c:4:S,W,E:198`
  # with byte-identical code: 326 meshes moved, the `peelrow` windrow
  # mirrored 40 m end for end, prims 6998 -> 6977, top_z 21.69 -> 20.76 m,
  # settle 1200 -> 1400 steps. That is why "re-bake the frozen kit record and
  # check it reproduces" could not be answered before this line existed.
  # Pinning the seed makes a bake reproducible from here on; it does NOT
  # make one match a bake taken before it.
  ENVS="ISAAC_SIM_HEADLESS=true PYTHONUNBUFFERED=1 PYTHONHASHSEED=0"
  ENVS="$ENVS FB_KIND=$KIND FB_NAME=$NAME FB_LEVEL=$LEVEL"
  ENVS="$ENVS FB_SEED=$SEED FB_BUILD_SEED=$BUILD_SEED FB_INDEX=$i"
  ENVS="$ENVS FB_ORIGIN=$ORIGIN FB_SIDES=$SIDES"
  ENVS="$ENVS FB_OUT=$CITY_OUT FB_BAKED_KITS=$FB_BAKED_KITS FB_VERIFY=1"
  ENVS="$ENVS FB_CITY_JSON=$CITY_JSON_C"
  [ -n "$SOOT_TEX_COMPRESS" ] && ENVS="$ENVS SOOT_TEX_COMPRESS=$SOOT_TEX_COMPRESS"
  # PER-LEVEL SETTLE BUDGET — see the header comment. F0-F3 (and any
  # unrecognised level, fail-safe) get REC_TIER=low/high; F4 gets "mid";
  # F5/F5c/F6 get the SETTLE_STEPS/SETTLE_QUIET value unchanged ("high").
  settle_for_level "$LEVEL"
  ENVS="$ENVS SETTLE_STEPS=$REC_STEPS SETTLE_QUIET=$REC_QUIET"
  ENVS="$ENVS SETTLE_DECOMP_M=$SETTLE_DECOMP_M SETTLE_FABRIC=$SETTLE_FABRIC"
  # THE KIT LOOK IS FROZEN, SO THE KIT KEEPS THE OLD SETTLE. Every other
  # kind gets `disaster.settle`'s corrected measurements (points-based
  # grade, net-travel rest, freeze-and-carry-on instead of giving up on a
  # stall, size-scaled convex decomposition). See settle.py's docstring.
  REST_V2=$SETTLE_REST_V2
  [ "$KIND" = "kit" ] && REST_V2=0
  ENVS="$ENVS SETTLE_REST_V2=$REST_V2"
  RUN="mkdir -p '$CLOGDIR' '$CITY_OUT'; : > '$CLOG'; cd /isaac-sim && env $ENVS PYTHONPATH=\"\$ISAAC_SIM_PYTHONPATH\" timeout ${TIMEOUT_S}s /isaac-sim/python.sh $LAUNCHER --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts --no-window > '$CLOG' 2>&1; echo \"EXIT \$?\" >> '$CLOG'"

  echo "=== [$i] $ENTRY -> $STEM  ($STATUS$([ "$FORCE" = 1 ] && [ "$STATUS" = "HAVE" ] && echo ", --force")) settle=$REC_TIER(${REC_STEPS}+${REC_QUIET})"
  echo "+ docker exec $CONTAINER bash -c \"$RUN\""
  if [ "$DRY" = 1 ]; then USDS+=("$CITY_OUT/$STEM.usd"); i=$((i+1)); continue; fi

  t0=$(date +%s)
  docker exec "$CONTAINER" bash -c "$RUN"
  dt=$(( $(date +%s) - t0 ))

  if docker exec "$CONTAINER" grep -q "FIRE BAKE DONE" "$CLOG" 2>/dev/null; then
    SZ=$(docker exec "$CONTAINER" bash -c "stat -c %s '$CITY_OUT/$STEM.usd' 2>/dev/null || echo 0")
    MB=$(awk -v b="$SZ" 'BEGIN{printf "%.1f", b/1e6}')
    VOK=$(docker exec "$CONTAINER" grep -c "BAKE VERIFY OK" "$CLOG" 2>/dev/null || echo 0)
    # A SETTLE THAT DID NOT COME TO REST IS A DEFECT OF THIS RECORD, and
    # until 2026-08-31 the only place it appeared was 500 lines down one
    # container log: the driver printed `verify-OK` over a bake that had
    # just deleted 80 bodies for being frozen mid-flight. `settle.run`
    # prints one greppable `REST_CHECK key=value` line per bake; this
    # scrapes it and puts the numbers on the record's own summary line.
    RC=$(docker exec "$CONTAINER" grep -h "REST_CHECK" "$CLOG" 2>/dev/null | tail -1 || true)
    AT_REST=$(printf '%s' "$RC" | sed -n 's/.*at_rest=\([0-9]*\).*/\1/p')
    SM=$(printf '%s' "$RC" | sed -n 's/.*still_moving=\([0-9]*\).*/\1/p')
    BGP=$(printf '%s' "$RC" | sed -n 's/.*below_grade_pts=\([0-9]*\).*/\1/p')
    FRZ=$(printf '%s' "$RC" | sed -n 's/.*frozen=\([0-9]*\).*/\1/p')
    RSN=$(printf '%s' "$RC" | sed -n 's/.*reason=\([a-z]*\).*/\1/p')
    if [ -z "$RC" ]; then
      REST="rest-?"
    elif [ "${AT_REST:-0}" = "1" ]; then
      REST="rest-OK$([ "${FRZ:-0}" != "0" ] && echo "(froze ${FRZ})")"
    else
      REST="NOT-AT-REST(${SM:-?} moving, ${BGP:-?} sunk, ${RSN:-?})"
      notrest=$((notrest+1))
    fi
    echo "    DONE in ${dt}s, ${MB} MB, verify=$([ "$VOK" -gt 0 ] && echo OK || echo PROBLEM), $REST"
    SUMMARY+=("$(printf '%-2s %-42s %-6s %7ss %8s MB  %-15s %s' "$i" "$STEM" "$LEVEL" "$dt" "$MB" "$([ "$VOK" -gt 0 ] && echo verify-OK || echo VERIFY-PROBLEM)" "$REST")")
    USDS+=("$CITY_OUT/$STEM.usd")
    ok=$((ok+1))
  else
    echo "    FAILED after ${dt}s — see $HLOG"
    docker exec "$CONTAINER" grep -E "Traceback|Error|FIRE BAKE FAILED|EXIT [1-9]" "$CLOG" 2>/dev/null | tail -12
    SUMMARY+=("$(printf '%-2s %-42s %-6s %7ss %8s     FAILED' "$i" "$STEM" "$LEVEL" "$dt" "-")")
    fail=$((fail+1))
  fi
  i=$((i+1))
done <<< "$CLASS_OUT"

[ "$DRY" = 1 ] && { echo; echo "(dry run — nothing was executed)"; }

echo
echo "========================================================================"
echo "FIRE CITY BAKE   seed $CITY_SEED   $ok baked, $skipped skipped, $fail failed, $notrest NOT AT REST, $(( $(date +%s) - t_all )) s total"
if [ ${#SUMMARY[@]} -gt 0 ]; then printf '%s\n' "${SUMMARY[@]}"; fi
if [ "$notrest" -gt 0 ]; then
  echo
  echo "  !! $notrest record(s) BAKED A PILE THAT WAS NOT AT REST."
  echo "  !! Bodies still moving at bake time are DELETED from the export by"
  echo "  !! fire_bake_launch_script.py, so those cells are missing debris."
  echo "  !! grep REST_CHECK $HOSTLOGDIR/city_${CITY_SEED}_<stem>.log"
fi
echo "  bakes:      $CONTAINER:$CITY_OUT"
echo "  city json:  $CONTAINER:$CITY_JSON_DIR"
echo "  logs:       $HOSTLOGDIR/city_${CITY_SEED}_<stem>.log"
echo "------------------------------------------------------------------------"
FA_LIST=$(IFS=,; echo "${USDS[*]-}")
[ -z "$FA_LIST" ] && FA_LIST="$CITY_OUT"
echo "FA_BAKES for the city assembly (manifest order):"
echo
echo "  FA_BAKES=$FA_LIST"
echo "========================================================================"
if [ "$FB_REST_STRICT" != "0" ] && [ "$notrest" -gt 0 ]; then exit 1; fi
[ "$fail" = 0 ]
