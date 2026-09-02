#!/usr/bin/env bash
# QUERY-COMBINATION SWEEP over the standing/buried A/B rig — run on the HOST
# while osmo/missions/raven_query_sweep_t250.yaml has the drone parked at the
# rig. For each combo it:
#   1. writes the vocabulary file (TARGET FIRST -> target = q0/sim_0) that
#      querying.query_file points at,
#   2. writes /tmp/offboard/sweep_env in the offboard container (probe
#      contrast set + the query_file hydra override, sourced by
#      offboard_compute.sh's supervisor before every mapper start),
#   3. kills the mapper (the supervisor restarts it on the new combo),
#   4. dwells while the parked drone re-maps the rig, then emits the probe
#      rows for the STANDER (1.21,-69.42) and the BURIED idx10 (-4.79,-69.42).
# Softmax stays on throughout (the mission's RAYFRONTS_COMPUTE_PROB=True).
#
# One line of output per event — run it under a Monitor.
set -uo pipefail
OC=offboard-compute
VOCAB_HOST=common/rayfronts_configs/sweep_vocab.txt
VOCAB_CONT=/root/AirStack/common/rayfronts_configs/sweep_vocab.txt
DWELL="${SWEEP_DWELL_S:-100}"
# Fixed candidate set so the solo columns stay comparable across combos.
PROMPTS="person,human,casualty,person lying down,human body"

# combo spec: name|target|comma-separated backgrounds
# LADDER 2 (2026-09-02): the deciding runs. L2a/L2b isolate the wording
# (casualty vs human, both debris-free); the _nopca twins disable the
# feature compressor entirely (~2k voxels fits uncompressed) to remove the
# per-incarnation single-frame PCA-basis lottery that sank c5.
# Field 4 = RAYFRONTS_CONFIG for the combo. shared_humans_nopca composes on
# top of shared_humans with the compressor fully nulled (the earlier
# ~feat_compressor CLI delete left the literal out_dim/path block behind and
# it was instantiated as a "compressor" — .is_fitted crash loop, 20:43).
COMBOS=(
  "L2b_casualty_nodebris_nopca|casualty|house, window, tree, grass, road|shared_humans_nopca"
  "L2c_human_nodebris_nopca|human|house, window, tree, grass, road|shared_humans_nopca"
  "L2d_person_nodebris_nopca|person|house, window, tree, grass, road|shared_humans_nopca"
)

run_combo() {
  local name="$1" target="$2" bgs="$3" cfg="${4:-shared_humans}"
  echo "=== COMBO $name: target='$target' bg='$bgs' ==="

  { echo "$target"; echo "$bgs" | tr ',' '\n' | sed 's/^ *//;s/ *$//' | grep -v '^$'; } > "$VOCAB_HOST"

  docker exec $OC bash -c "cat > /tmp/offboard/sweep_env << EOF
export RAYFRONTS_PROBE_BACKGROUND=\"$bgs\"
export RAYFRONTS_PROBE_PROMPTS=\"$PROMPTS\"
export RAYFRONTS_CONFIG=\"$cfg\"
export RF_SWEEP_OVERRIDES=\"querying.query_file=$VOCAB_CONT\"
EOF"

  local mark
  mark=$(docker exec $OC bash -c "wc -l < /tmp/offboard/rayfronts_mapping.log")
  docker exec $OC bash -c "pkill -TERM -f multi_robot_mapping_server" >/dev/null 2>&1

  local t0=$SECONDS ok=0
  while [ $((SECONDS - t0)) -lt 120 ]; do
    if docker exec $OC bash -c "tail -n +$mark /tmp/offboard/rayfronts_mapping.log | grep -q 'Received queries'"; then
      ok=1; break
    fi
    sleep 5
  done
  if [ "$ok" != 1 ]; then
    echo "COMBO $name: mapper never seeded queries within 120s — skipping (check the log)"
    return 1
  fi
  docker exec $OC bash -c "tail -n +$mark /tmp/offboard/rayfronts_mapping.log | grep -m1 'Received queries'"

  sleep "$DWELL"

  echo "--- $name STANDER (1.21,-69.42) ---"
  docker exec $OC bash -c "tail -n +$mark /tmp/offboard/rayfronts_mapping.log | grep -A1 'probe@(1.21' | tail -4"
  echo "--- $name BURIED idx10 (-4.79,-69.42) ---"
  docker exec $OC bash -c "tail -n +$mark /tmp/offboard/rayfronts_mapping.log | grep -A1 'probe@(-4.79' | tail -4"
  echo "--- $name map-wide (FP pressure: >0.5 count on the TARGET row) ---"
  docker exec $OC bash -c "tail -n +$mark /tmp/offboard/rayfronts_mapping.log | grep '\[raw vox\]' | tail -7"
  docker exec $OC bash -c "tail -n +$mark /tmp/offboard/rayfronts_mapping.log | grep -E '\[oom-restart\]|OutOfMemory' | tail -1"
  echo "=== COMBO $name done ==="
}

echo "SWEEP START $(date +%H:%M:%S) — ${#COMBOS[@]} combos, dwell ${DWELL}s"
for spec in "${COMBOS[@]}"; do
  IFS='|' read -r name target bgs cfg <<< "$spec"
  run_combo "$name" "$target" "$bgs" "$cfg"
done
echo "SWEEP DONE $(date +%H:%M:%S). Last combo's sweep_env is still active;"
echo "rm it in the container (or write the winner) before a normal mission:"
echo "  docker exec offboard-compute rm /tmp/offboard/sweep_env"
