# Wiring snapshot goldens

Committed observed-wiring baselines for the drift check in
[`tests/system/test_wiring_snapshot.py`](../../system/test_wiring_snapshot.py)
(RFC #379 §4.4: the wiring picture is snapshotted from the **running** ROS
graph — observed, never generated from configuration — so it cannot lie or
rot).

## Files

One golden per `(sim, num_robots)` configuration, named

```text
full_default.<sim>.<num_robots>robot.md
```

e.g. `full_default.isaacsim.1robot.md`, `full_default.msairsim.3robot.md`.
Each file is a full `wiring.md`: provenance lines, a mermaid dataflow diagram
(nodes grouped by namespace, edges labeled topic/type), and a
machine-readable JSON trailer (`<!-- wiring-graph-v1 ... -->`) that the drift
check actually compares. **Never hand-edit these files** — they are only ever
copied from a validated run.

## Bootstrap: committing the first golden

Goldens are committed from a validated local run:

1. Run the wiring mark against the target configuration, e.g.

   ```bash
   airstack test -m wiring --sim isaacsim --num-robots 1 -v
   ```

2. With no golden present, the test **passes** and writes the observed
   snapshot to `tests/results/<timestamp>/wiring/observed_full_default.md`,
   logging an `INSTRUCTION:` line with the exact destination path.

3. Validate the observed snapshot (skim the mermaid diagram: expected nodes
   present, no junk endpoints), then copy it to the golden name and commit:

   ```bash
   cp tests/results/<timestamp>/wiring/observed_full_default.md \
      tests/goldens/wiring/full_default.isaacsim.1robot.md
   ```

## Drift check semantics

Once a golden exists, the test snapshots the running graph and diffs the two
normalized graphs (`tests/wiring_snapshot.py diff`): missing/extra nodes,
missing/extra pub/sub edges, per-edge QoS mismatches, and per-topic type
mismatches all fail the test with a JSON verdict. Infra noise
(`/parameter_events`, `/rosout`, pid-suffixed `launch_ros_*` /
`transform_listener_impl_*` / `_ros2cli*` nodes) is normalized out on both
sides; `/tf` and `/tf_static` are kept — frame plumbing is wiring.

A PR that intentionally changes wiring must regenerate the affected goldens
(same flow as bootstrap) and commit them, so the review diff shows the
topology change.
