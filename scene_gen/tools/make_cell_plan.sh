#!/usr/bin/env bash
# make_cell_plan.sh — the HOST half of an urban-fire cell: everything that can
# be decided, measured and REVIEWED without a GPU.
#
#   bash scene_gen/tools/make_cell_plan.sh                 # all three levels
#   LEVELS="1" REVIEW_DIR=~/layout_review bash scene_gen/tools/make_cell_plan.sh
#
# Produces, per level N:
#   scene_gen/_plans/fc_dump_1km_l<N>.json    the frozen layout (dump v1)
#   scene_gen/_plans/fire_corr_l<N>.json      the corridor fire manifest
#   scene_gen/_plans/bake_l<N>.json           the lazy-bake worklist for the pod
#   <REVIEW_DIR>/L<N>_*.png                   districts / diversity / blockshapes / damage
#
# Then hand the pod `scene_gen/tools/urban_fire_cell.sh`, which consumes
# exactly those three JSON files.
#
# WHY THE SPLIT. A 1 km city takes ~90 s to lay out on CPU and a few minutes
# to solve a fire over. It takes hours on a pod to bake and assemble. So every
# decision that can be made from arithmetic — which seed, which district mix,
# how repetitive, where the fire runs, how much of the city burns, what needs
# baking — is made and reviewed here, and the pod only ever sees a plan that
# has already passed. The review PNGs are the gate: a cell nobody has looked
# at in 2D should not cost a pod.
set -uo pipefail

cd "$(dirname "$0")/../.." || exit 1
LEVELS="${LEVELS:-1 2 3}"
REVIEW_DIR="${REVIEW_DIR:-$HOME/layout_review}"
PLANS="scene_gen/_plans"

# Must match the pod. See `urban_fire_cell.sh`'s determinism note: the layout
# desyncs between stages if these differ, and the symptom appears much later.
export PYTHONHASHSEED=0
export SG_INSTANCE_PLACEMENTS=1

mkdir -p "$REVIEW_DIR" "$PLANS"
say () { printf '\n\033[1m== %s ==\033[0m\n' "$*"; }
die () { printf '\033[31mFAILED: %s\033[0m\n' "$*" >&2; exit 1; }

py () { python3 "$@"; }

for L in $LEVELS; do
  PRESET="downtown_urban_fire_1000_l${L}"
  YAML="scene_gen/config/presets/${PRESET}.yaml"
  [ -f "$YAML" ] || die "no preset $YAML — run tools/make_urban_fire_levels.py first"

  # Every knob comes off the preset, so the preset stays the single source of
  # truth and this script has no numbers of its own to drift from it.
  read -r SEED FRAC EPOCH COLLAPSE F6 <<EOF
$(py - "$YAML" <<'PY'
import sys, yaml
d = yaml.safe_load(open(sys.argv[1]))
print(d["seed"],
      d["fire_target_frac"],
      float(d["duration_s"]) * float(d["start_offset_frac"]),
      d["fire_collapse"],
      d["fire_f6"])
PY
)
EOF

  say "LEVEL ${L}  seed=${SEED}  area=${FRAC}  epoch=${EPOCH}s"

  say "1/4 layout -> dump"
  py scene_gen/tools/plan_to_fc_dump.py \
      --config "$PRESET" --fire-layout --seed "$SEED" --region 1000 \
      --out "$PLANS/fc_dump_1km_l${L}.json" --warn-guessed \
    | grep '^\[fc-dump\]' || die "dump (level $L)"

  say "2/4 corridor fire -> manifest + bake worklist"
  py scene_gen/tools/fire_corridor_manifest.py \
      --dump "$PLANS/fc_dump_1km_l${L}.json" \
      --target-frac "$FRAC" --epoch-s "$EPOCH" \
      --collapse "$COLLAPSE" --f6 "$F6" --seed "$SEED" \
      --out "$PLANS/fire_corr_l${L}.json" \
      --bake-list "$PLANS/bake_l${L}.json" \
    | grep -vE '^\[(scene_gen|districts|city_layout|compile_disaster|parks)\]' \
    || die "corridor (level $L)"

  say "3/4 review sheets -> $REVIEW_DIR"
  py scene_gen/tools/layout_review_png.py \
      --config "$PRESET" --fire-layout --seed "$SEED" --region 1000 \
      --manifest "$PLANS/fire_corr_l${L}.json" --shapes --stats-json \
      --out-dir "$REVIEW_DIR" --tag "L${L}_s${SEED}" \
    | grep '^\[review\]' || die "review (level $L)"

  # OCCUPIED-BLOCK GATE. The shipped defect this exists to catch was "~54 % of
  # blocks within the window carry zero buildings" -- a number no review PNG
  # makes obvious and no other gate measures. `_stats.json` already carries
  # the block census, so assert on it here rather than trusting an eyeball on
  # the districts sheet.
  say "4/5 occupied-block gate"
  py - "$REVIEW_DIR/L${L}_s${SEED}_stats.json" <<'BLOCKGATE' || die "block occupancy (level $L)"
import json, sys
d = json.load(open(sys.argv[1]))
blocks = int(d.get("blocks") or 0)
houses = int(d.get("houses") or 0)
if blocks <= 0:
    sys.exit("no blocks in %s" % sys.argv[1])
if houses < 250:
    sys.exit("only %d buildings across %d blocks -- a 1 km downtown should "
             "carry 400+" % (houses, blocks))
print("  %d buildings across %d blocks (%.1f per block)"
      % (houses, blocks, houses / float(blocks)))
BLOCKGATE

  say "5/5 offline gates"
  py -m pytest -q scene_gen/tests/test_shared_urban_layout.py -m "not slow" \
    | tail -2 || die "gates (level $L)"
done

say "PLAN COMPLETE"
cat <<EOF

Review these before spending a pod:
  $REVIEW_DIR/L*_districts.png    zoning, every block labelled
  $REVIEW_DIR/L*_diversity.png    repeats within 60 m ringed in red
  $REVIEW_DIR/L*_blockshapes.png  block-size monotony per district
  $REVIEW_DIR/L*_damage.png       where the fire runs and how hard

Then, on the pod:
  bash scene_gen/tools/urban_fire_cell.sh        # LEVELS="1 2 3" for all

Bake worklists (what the pod actually has to bake):
EOF
for L in $LEVELS; do
  py - "$PLANS/bake_l${L}.json" <<'PY'
import sys, json, os
try:
    d = json.load(open(sys.argv[1]))
except Exception:
    raise SystemExit
print("  level %-3s %3d assets / %3d instances"
      % (os.path.basename(sys.argv[1])[6:-5], d["n_assets"], d["n_instances"]))
PY
done
