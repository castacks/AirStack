#!/usr/bin/env bash
# BACKGROUND-QUERY sweep — target fixed at `person`, only the background set
# varies. Motivation (user, live 2026-09-02): large FP areas mean clutter
# patches whose true label is MISSING from the background set — with nowhere
# to put their vote, they softmax onto person. Each combo measures:
#   * FP pressure: count/where of sim_0 > 0.5/0.6/0.8 over the whole
#     published cloud (fp_pressure.py inside the robot container),
#   * TP references: the probe rows at the STANDER (1.21,-69.42) and the
#     BURIED idx10 (-4.79,-69.42), so a set that kills FPs by killing people
#     is caught immediately.
# Run while osmo/missions/raven_query_sweep_t250.yaml has the drone parked.
set -uo pipefail
OC=offboard-compute
RC=disaster-dataset-robot-desktop-1
VOCAB_HOST=common/rayfronts_configs/sweep_vocab.txt
VOCAB_CONT=/root/AirStack/common/rayfronts_configs/sweep_vocab.txt
DWELL="${SWEEP_DWELL_S:-100}"
PROMPTS="person,human,casualty,person lying down,human body"

# name|comma-separated background set (target `person` is prepended)
COMBOS=(
  "B1_baseline|road, grass, tree, house, debris, sky"
  "B2_roof_wall|road, grass, tree, house, roof, wall, debris, sky"
  "B3_fence_car|road, grass, tree, house, fence, car, debris, sky"
  "B4_rubble_wreck|road, grass, tree, house, debris, rubble, wreckage, sky"
  "B5_dirt_ground|road, dirt, ground, grass, tree, house, debris, sky"
  "B6_kitchen_sink|road, grass, tree, house, roof, wall, fence, car, dirt, rubble, wreckage, debris, sky"
)

docker cp scripts/raven_live/fp_pressure.py $RC:/tmp/fp_pressure.py

run_combo() {
  local name="$1" bgs="$2"
  echo "=== BG-COMBO $name: person | $bgs ==="
  { echo "person"; echo "$bgs" | tr ',' '\n' | sed 's/^ *//;s/ *$//' | grep -v '^$'; } > "$VOCAB_HOST"
  docker exec $OC bash -c "cat > /tmp/offboard/sweep_env << EOF
export RAYFRONTS_PROBE_BACKGROUND=\"$bgs\"
export RAYFRONTS_PROBE_PROMPTS=\"$PROMPTS\"
export RAYFRONTS_CONFIG=shared_humans_nopca
export RF_SWEEP_OVERRIDES=\"querying.query_file=$VOCAB_CONT\"
EOF"
  local mark
  mark=$(docker exec $OC bash -c "wc -l < /tmp/offboard/rayfronts_mapping.log")
  docker exec $OC bash -c "pkill -TERM -f multi_robot_mapping_server" >/dev/null 2>&1
  local t0=$SECONDS ok=0
  while [ $((SECONDS - t0)) -lt 120 ]; do
    if docker exec $OC bash -c "tail -n +$mark /tmp/offboard/rayfronts_mapping.log | grep -q 'Received queries'"; then ok=1; break; fi
    sleep 5
  done
  [ "$ok" = 1 ] || { echo "BG-COMBO $name: mapper never seeded — skipping"; return 1; }
  docker exec $OC bash -c "tail -n +$mark /tmp/offboard/rayfronts_mapping.log | grep -m1 'Received queries'"
  sleep "$DWELL"
  echo "--- $name FP pressure (whole published cloud) ---"
  docker exec $RC bash -c "set +u; source /opt/ros/jazzy/setup.bash; set -u; export ROS_DOMAIN_ID=1; timeout 70 python3 /tmp/fp_pressure.py 1 60" 2>/dev/null
  echo "--- $name STANDER ---"
  docker exec $OC bash -c "tail -n +$mark /tmp/offboard/rayfronts_mapping.log | grep -A1 'probe@(1.21' | tail -4"
  echo "--- $name BURIED idx10 ---"
  docker exec $OC bash -c "tail -n +$mark /tmp/offboard/rayfronts_mapping.log | grep -A1 'probe@(-4.79' | tail -4"
  docker exec $OC bash -c "tail -n +$mark /tmp/offboard/rayfronts_mapping.log | grep '\[raw top\] person' | tail -1"
  echo "=== BG-COMBO $name done ==="
}

echo "BG SWEEP START $(date +%H:%M:%S) — ${#COMBOS[@]} combos, dwell ${DWELL}s"
for spec in "${COMBOS[@]}"; do
  IFS='|' read -r name bgs <<< "$spec"
  run_combo "$name" "$bgs"
done
echo "BG SWEEP DONE $(date +%H:%M:%S) — last combo still active in sweep_env."
