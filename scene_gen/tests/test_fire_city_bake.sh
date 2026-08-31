#!/usr/bin/env bash
# test_fire_city_bake.sh — offline smoke test for `scene_gen/tools/
# fire_city_bake.sh` + `scene_gen/tools/fire_city_manifest.py`
# (`urban_fire_city_plan.md` sec 3, work item #6). NO docker, NO GPU, NO
# Kit: a fake 3-record manifest, one stem pre-created as an empty
# `.usd`+`.json` pair, run through `--dry-run` (which never checks the
# container is up), asserting the HAVE/NEED classification and the printed
# `docker exec` lines for the two NEED entries.
#
#     bash scene_gen/tests/test_fire_city_bake.sh

set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
TOOLS="$REPO/scene_gen/tools"

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

FAIL=0
ok()   { echo "  ok    $1"; }
bad()  { echo "  FAIL  $1"; FAIL=$((FAIL+1)); }
assert_contains() {
  # $1 = haystack file, $2 = needle, $3 = description
  if grep -qF -- "$2" "$1"; then ok "$3"; else bad "$3 (missing: $2)"; fi
}
assert_not_contains() {
  if grep -qF -- "$2" "$1"; then bad "$3 (unexpectedly present: $2)"; else ok "$3"; fi
}

echo "=== bash -n both scripts ==="
if bash -n "$TOOLS/fire_city_bake.sh"; then ok "fire_city_bake.sh parses"; else bad "fire_city_bake.sh: bash -n failed"; fi
if python3 -c "import ast; ast.parse(open('$TOOLS/fire_city_manifest.py').read())" 2>/dev/null
then ok "fire_city_manifest.py parses"; else bad "fire_city_manifest.py: parse failed"; fi
echo

# ---------------------------------------------------------------------------
# The fake manifest: 3 records, city seed 4242.
#   [0] gac SM_Building_02 F1   seed 4242  -- pre-baked (HAVE)
#   [1] kit commercial_mid F5c  seed 4273  -- not baked  (NEED)
#   [2] dtc Building_12   F2    seed 4304  -- not baked  (NEED)
# Seeds follow damaged_manifest's own `seed_base + 31*i` convention (not
# load-bearing for this test, just realistic).
# ---------------------------------------------------------------------------
MANIFEST="$TMP/fire_city_4242.json"
cat > "$MANIFEST" <<'JSON'
{
  "seed": 4242,
  "manifest": [
    {"i": 0, "kind": "gac", "asset": "SM_Building_02", "style": null,
     "level": "F1", "origin": null, "sides": [], "seed": 4242,
     "cell": "/World/stage/generated/house_0_1", "x": 10.0, "y": 5.0,
     "yaw_deg": 90.0, "z": 0.0, "typology": "lowrise",
     "usd": "omniverse://x/SM_Building_02.usd"},
    {"i": 1, "kind": "kit", "asset": null, "style": "commercial_mid",
     "level": "F5c", "origin": 1, "sides": ["S", "E"], "seed": 4273,
     "cell": "/World/stage/generated/house_1_2", "x": 40.0, "y": 12.0,
     "yaw_deg": 0.0, "z": 0.0, "typology": "midrise",
     "usd": "omniverse://x/mce.usd"},
    {"i": 2, "kind": "dtc", "asset": "Building_12", "style": null,
     "level": "F2", "origin": 0, "sides": ["W"], "seed": 4304,
     "cell": "/World/stage/generated/house_2_3", "x": 70.0, "y": 20.0,
     "yaw_deg": 180.0, "z": 0.0, "typology": "brick_midrise",
     "usd": "omniverse://x/Building_12.usdc"}
  ]
}
JSON

FB_OUT_HOST="$TMP/fire_bakes"
CITY_DIR="$FB_OUT_HOST/city_4242"
mkdir -p "$CITY_DIR"

# --- entry/stem for record 0, computed the SAME way the driver does, so the
# pre-created filenames are the exact cache key (not a guess at the format).
STEM0=$(python3 "$TOOLS/fire_city_manifest.py" "$MANIFEST" --out-dir "$CITY_DIR" \
       | awk -F'\t' 'NR==1{print $2}')
if [ -z "$STEM0" ]; then
  echo "FATAL: could not compute record 0's stem via fire_city_manifest.py" >&2
  exit 1
fi
: > "$CITY_DIR/$STEM0.usd"
: > "$CITY_DIR/$STEM0.json"

echo "=== fire_city_manifest.py: HAVE/NEED classification ==="
CLASS="$TMP/classify.out"
python3 "$TOOLS/fire_city_manifest.py" "$MANIFEST" --out-dir "$CITY_DIR" > "$CLASS"
cat "$CLASS" | sed 's/^/  /'
assert_contains "$CLASS" "$(printf '%s\tHAVE' "$STEM0")" "record 0 ($STEM0) classified HAVE"
N_NEED=$(grep -c $'\tNEED$' "$CLASS")
if [ "$N_NEED" = 2 ]; then ok "exactly 2 records classified NEED"; else bad "expected 2 NEED, got $N_NEED"; fi
assert_contains "$CLASS" "SUMMARY total=3 have=1 stale=0 need=2 error=0" "summary line matches"
echo

echo "=== fire_city_bake.sh --dry-run: skip HAVE, plan NEED ==="
RUN="$TMP/run.out"
FB_OUT="$FB_OUT_HOST" bash "$TOOLS/fire_city_bake.sh" "$MANIFEST" --dry-run > "$RUN" 2>&1
RC=$?
cat "$RUN" | sed 's/^/  /'
if [ "$RC" = 0 ]; then ok "--dry-run exits 0"; else bad "--dry-run exited $RC"; fi
assert_contains "$RUN" "city seed 4242" "resolved the city seed from the manifest's top-level field"
assert_contains "$RUN" "$STEM0" "record 0's stem appears in the plan"
assert_contains "$RUN" "(HAVE, skipping)" "record 0 is reported as skipped (HAVE)"

N_DOCKER=$(grep -c "^+ docker exec" "$RUN")
if [ "$N_DOCKER" = 2 ]; then
  ok "exactly 2 'docker exec' bake lines printed (the two NEED entries)"
else
  bad "expected 2 'docker exec' bake lines, got $N_DOCKER"
fi
assert_contains "$RUN" "FB_KIND=kit FB_NAME=commercial_mid FB_LEVEL=F5c" "NEED entry 1 (kit) env is correct"
assert_contains "$RUN" "FB_KIND=dtc FB_NAME=Building_12 FB_LEVEL=F2" "NEED entry 2 (dtc) env is correct"
assert_not_contains "$RUN" "FB_KIND=gac FB_NAME=SM_Building_02" "the HAVE (gac) entry never got a docker exec line"
assert_contains "$RUN" "FB_OUT=$CITY_DIR" "FB_OUT points at the city-scoped subdir"
assert_contains "$RUN" "(dry run — nothing was executed)" "dry-run banner printed"
assert_contains "$RUN" "0 baked, 1 skipped, 0 failed" "summary: 0 baked / 1 skipped / 0 failed"
echo

echo "=== fire_city_bake.sh --dry-run --force: HAVE entry is planned too ==="
RUN2="$TMP/run_force.out"
FB_OUT="$FB_OUT_HOST" bash "$TOOLS/fire_city_bake.sh" "$MANIFEST" --dry-run --force > "$RUN2" 2>&1
N_DOCKER2=$(grep -c "^+ docker exec" "$RUN2")
if [ "$N_DOCKER2" = 3 ]; then
  ok "--force plans all 3 entries (3 'docker exec' lines)"
else
  bad "expected 3 'docker exec' lines under --force, got $N_DOCKER2"
fi
echo

echo "=== a second run sees the SAME manifest as all-HAVE once every stem exists ==="
# Bake nothing for real -- just pre-create the other two stems the way the
# first run's plan named them, then reclassify.
python3 "$TOOLS/fire_city_manifest.py" "$MANIFEST" --out-dir "$CITY_DIR" > "$CLASS"
awk -F'\t' '$3=="NEED"{print $2}' "$CLASS" | while read -r stem; do
  : > "$CITY_DIR/$stem.usd"
  : > "$CITY_DIR/$stem.json"
done
RUN3="$TMP/run_all_have.out"
FB_OUT="$FB_OUT_HOST" bash "$TOOLS/fire_city_bake.sh" "$MANIFEST" --dry-run > "$RUN3" 2>&1
N3=$(grep -c "^+ docker exec" "$RUN3" || true)
if [ "${N3:-1}" = 0 ]; then
  ok "no 'docker exec' lines once every stem is cached"
else
  bad "expected 0 'docker exec' lines, got ${N3:-?}"
fi
assert_contains "$RUN3" "0 baked, 3 skipped, 0 failed" "summary: 0 baked / 3 skipped / 0 failed"
echo

echo "========================================================================"
if [ "$FAIL" = 0 ]; then
  echo "test_fire_city_bake.sh: ALL CHECKS PASSED"
else
  echo "test_fire_city_bake.sh: $FAIL CHECK(S) FAILED"
fi
exit $([ "$FAIL" = 0 ] && echo 0 || echo 1)
