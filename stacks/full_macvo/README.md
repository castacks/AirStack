# `full_macvo` — trunk reference stack

Full autonomy with **MAC-VO** as the disparity source for the local planner.
Local-planner variants are expressed as named stacks a few include lines
apart — rather than as launch-file arguments — so each variant is directly
selectable and carries its own observed wiring baseline.

**Requires: `airstack module add asm_macvo`.** MAC-VO is not trunk-resident —
the `macvo_ros2` package, its Python/TensorRT dependencies, and the model
weights all ship in the [asm_macvo](https://github.com/castacks/asm_macvo)
module. Until the module is synced (`modules.repos` pins it),
`$(find-pkg-share macvo_ros2)` in this stack's launch file will not resolve
and bring-up fails at the macvo include.

## What it launches

Identical to [`full_default`](../full_default/README.md) except:

1. The module-provided `macvo_ros2/launch/macvo.launch.xml` is included under
   the `perception` namespace, so the `macvo_ros2` node runs and publishes
   `/$ROBOT_NAME/perception/macvo/{odometry,point_cloud,disparity}` (all
   canonical-default args — zero remaps).
2. The `droan_gl.launch.xml` include passes
   `droan_gl_disparity_topic:=/$ROBOT_NAME/perception/macvo/disparity`,
   wiring the planner's disparity input to MAC-VO's real output topic.

## How to run

```bash
# One-time: pull the asm_macvo module and build its dependency layer
airstack module add asm_macvo
airstack module sync
airstack module lock --build

airstack up --stack full_macvo --sim isaac --robots 1
airstack ready
```

## Known limits

- Every layer is composed module-by-module in `stack.launch.xml`, except
  `interface.launch.py` (wrapped by design — the safety boundary) and
  `logging.launch.xml` (already a single self-contained module).
- `stereo_image_proc` runs alongside MAC-VO (its module include is kept in
  `stack.launch.xml`): the stereo point cloud feeds other consumers, so this
  stack runs both estimators. A leaner macvo-only preset can drop that
  include once downstream consumers are audited.
- MAC-VO is GPU-heavy; expect reduced sim real-time factor on a shared GPU.
- The committed `wiring.md` — captured from this stack's own first validated
  snapshot run — is the baseline.
- `modules.repos` pins `asm_macvo`; `docker-compose.yaml` stays an empty stub
  until `airstack module lock --build` generates the per-module compose
  override.

## wiring.md

This stack's observed wiring diagram is committed at [wiring.md](wiring.md);
CI drift-checks the running graph against it. Regenerate via
`airstack test -m wiring --stack full_macvo`.
