# `full_macvo` — trunk reference stack

Full autonomy with **MAC-VO** as the disparity source for the **DROAN**
stereo-disparity local planner. Local-planner variants are expressed as named
stacks a few include lines apart — rather than as launch-file arguments — so
each variant is directly selectable and carries its own observed wiring
baseline.

**Requires two modules**, both pinned in this stack's `modules.repos` and
synced by `airstack up` when missing: [asm_macvo](https://github.com/castacks/asm_macvo)
(the `macvo_ros2` package, its Python/TensorRT dependencies and model
weights) and [asm_droan](https://github.com/castacks/asm_droan) (the
`droan_gl` planner — this stack keeps DROAN rather than the lidar-fed MIGHTY
default because a disparity source is exactly what it exercises). Until both
are synced, the `$(find-pkg-share ...)` lookups in the launch file will not
resolve and bring-up fails at those includes.

## What it launches

Identical to [`full_droan`](../full_droan/README.md) (the GPU DROAN
topology) except:

1. The module-provided `macvo_ros2/launch/macvo.launch.xml` is included under
   the `perception` namespace, so the `macvo_ros2` node runs and publishes
   `/$ROBOT_NAME/perception/macvo/{odometry,point_cloud,disparity}` (all
   canonical-default args — zero remaps).
2. The `droan_gl.launch.xml` include passes
   `droan_gl_disparity_topic:=/$ROBOT_NAME/perception/macvo/disparity`,
   wiring the planner's disparity input to MAC-VO's real output topic.

## How to run

```bash
airstack up --stack full_macvo --sim isaac --robots 1   # syncs both pins, composes the dep layers
airstack ready
```

Explicitly, the one-time steps `airstack up` performs when the modules are
missing:

```bash
airstack module add https://github.com/castacks/asm_macvo.git --version <pin>
airstack module add https://github.com/castacks/asm_droan --version v0.1.0
airstack module lock --build      # MAC-VO's tier-2 layer + droan_gl's tier-1 apt layer
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
- `modules.repos` pins `asm_macvo` and `asm_droan`; `docker-compose.yaml`
  stays an empty stub — `airstack module lock --build` generates the
  per-module compose override.

## wiring.md

This stack's observed wiring diagram is committed at [wiring.md](wiring.md);
CI drift-checks the running graph against it. Regenerate via
`airstack test -m wiring --stack full_macvo`.
